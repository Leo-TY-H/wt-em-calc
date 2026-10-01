import unittest
from em_rejection_report import summarize

class Summary(unittest.TestCase):
    def test_mixed_and_unevaluated_counts(self):
        points=[dict(valid=True,reasons=[]),dict(valid=False,converged=True,reasons=['post-stall','post-stall']),dict(valid=False,converged=False,reasons=['trim did not converge','wing force limit']),dict(valid=False,not_evaluated=True,reasons=['VNE']),dict(valid=False,reasons=['future reason'])]
        row=summarize(dict(aircraft=[dict(name='sample',points=points)]))['aircraft'][0]
        self.assertEqual(row['sample_count'],5)
        self.assertEqual(row['rejected_count'],4)
        self.assertEqual(row['rejection_percent'],80.)
        self.assertEqual(row['reasons']['post-stall']['count'],1)
        self.assertEqual(row['categories'],dict(physical=2,**{'numerical+physical':1,'unclassified':1}))
        self.assertEqual(row['not_evaluated_rejected_count'],1)
    def test_empty(self):
        row=summarize(dict(aircraft=[dict(points=[])]))['aircraft'][0]
        self.assertIsNone(row['rejection_percent'])

if __name__=='__main__':unittest.main()
